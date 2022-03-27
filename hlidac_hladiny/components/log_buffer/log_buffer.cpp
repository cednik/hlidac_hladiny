#include "log_buffer.hpp"

#include <esp_err.h>
#include <esp_log.h>
#include <soc/soc_caps.h>
#include <soc/uart_reg.h>

#include <cstdio>

static const char* LOG_TAG = "LOG_BUF";

LogBuffer::mutex_t LogBuffer::_mutex {};
LogBuffer::WorkerType LogBuffer::_worker {};
vprintf_like_t LogBuffer::_previous_sink = nullptr;
LogBuffer::Log* LogBuffer::_log = nullptr;
size_t LogBuffer::_log_index = 0;
size_t LogBuffer::_message_size = 0;
TickType_t LogBuffer::_timeout = 0;
LogBuffer::StreamType LogBuffer::_stream_type = LogBuffer::StreamType::CSTREAM;
LogBuffer::StreamStorage LogBuffer::_stream = { .cstream = stdout };

bool LogBuffer::begin(
        size_t message_size,
        uint32_t queue_size,
        uint32_t stack_size,
        UBaseType_t task_priority,
        TickType_t timeout )
{
    if (isReady())
        end();
    std::lock_guard<mutex_t> lock (_mutex);
    _message_size = message_size;
    _timeout = timeout;
    _worker.queue_size(queue_size)
           .stack_size(stack_size)
           .task_priority(task_priority)
           .name(LOG_TAG)
           .process( [&](const Event::Type& type, const Event& data, void*) {
                switch (type) {
                case Event::Types::MSG:
                    switch (_stream_type)
                    {
                    case StreamType::ARDUINO_PRINT:
                        _stream.arduino->write(reinterpret_cast<const uint8_t*>(data.pLog->text), data.pLog->size);
                        break;
                    case StreamType::CSTREAM:
                        fwrite(data.pLog->text, sizeof(char), data.pLog->size, _stream.cstream);
                        break;
                    }
                    break;
                case Event::Types::REDIRECT:
                    _stream_type = data.stream_type;
                    _stream = data.stream;
                    break;
                }
            } );
    if (!_worker.begin()) {
        ESP_LOGE(LOG_TAG, "Can not start worker process.");
        return false;
    }
    _log = new Log[queue_size];
    if (!_log) {
        ESP_LOGE(LOG_TAG, "Can not allocate log buffer.");
        end();
        return false;
    }
    for (size_t i = 0; i != queue_size; ++i) {
        _log[i].size = 0;
        _log[i].text = new char[message_size];
        if (!_log[i].text) {
            if (i == 0) {
                ESP_LOGE(LOG_TAG, "Can not allocate any log text buffer.");
                end();
                return false;
            } else {
                ESP_LOGW(LOG_TAG, "Can not allocate log[%u] text buffer.", i);
                _worker.queue_size(i);
                break;
            }
        }
        _log[i].text[0] = 0;
    }
    _log_index = 0;
    _previous_sink = esp_log_set_vprintf(_vprintf);
    ESP_LOGI(LOG_TAG, "Ready");
    return true;
}

bool LogBuffer::end(TickType_t timeout) {
    std::lock_guard<mutex_t> lock (_mutex);
    if (!isReady())
        return false;
    esp_log_set_vprintf(_previous_sink);
    if (!_worker.end()) {
        return false;
    }
    _previous_sink = nullptr;
    for (size_t i = 0; i != _worker.queue_size(); ++i)
        delete[] _log[i].text;
    delete[] _log;
    _log = nullptr;
    ESP_LOGI(LOG_TAG, "End");
    return true;
}

bool LogBuffer::isReady() {
    return _worker.isAlive();
}

void LogBuffer::pause() {
    std::lock_guard<mutex_t> lock (_mutex);
    _worker.pause();
}
void LogBuffer::resume() {
    std::lock_guard<mutex_t> lock (_mutex);
    _worker.resume();
}

bool LogBuffer::redirect(StreamType type, StreamStorage stream) {
    std::lock_guard<mutex_t> lock (_mutex);
    if (_worker.isPaused()) {
        _stream_type = type;
        _stream = stream;
    } else {
        if (!_worker.query(Event::Types::REDIRECT, Event(type, stream))) {
            ESP_LOGE(LOG_TAG, "Can not send redirection event.");
            return false;
        }
    }
    return true;
}
bool LogBuffer::redirect(Print& stream) {
    StreamStorage s = { .arduino = &stream };
    return redirect(StreamType::ARDUINO_PRINT, s);
}
bool LogBuffer::redirect(FILE* stream) {
    StreamStorage s = { .cstream = stream };
    return redirect(StreamType::CSTREAM, s);
}

int LogBuffer::_vprintf(const char* format, va_list args) {
    std::lock_guard<mutex_t> lock (_mutex);
    Log* pLog = _log + _log_index;
    int res = vsnprintf(pLog->text, _message_size, format, args);
    if (res > 0) {
        pLog->size = res;
        _worker.send(Event::Types::MSG, Event(pLog));
        if (++_log_index >= _worker.queue_size())
            _log_index = 0;
    }
    return res;
}