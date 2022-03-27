#pragma once

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <worker.hpp>

#include <mutex>

class LogBuffer {
    typedef std::recursive_mutex mutex_t;

    enum class StreamType { ARDUINO_PRINT, CSTREAM };
    union StreamStorage {
        Print* arduino;
        FILE* cstream;
    };
    struct Log {
        size_t size;
        char*  text;
    };
    struct Event {
        typedef int Type;
        enum Types { REDIRECT, MSG };
        Event()
            : pLog{nullptr}
        {}
        Event(Log* _pLog)
            : pLog{_pLog}
        {}
        Event(StreamType type, StreamStorage _stream)
            : stream_type{type},
              stream{_stream}
        {}
        union {
            Log* pLog;
            struct {
                StreamType stream_type; 
                StreamStorage stream;
            };
        };
    };
    typedef Worker<Event, void, Event::Type> WorkerType;

    LogBuffer() = delete;
    LogBuffer(LogBuffer&) = delete;
public:

    static bool begin(
        size_t message_size = 256,
        uint32_t queue_size = 64,
        uint32_t stack_size = 2048,
        UBaseType_t task_priority = configMAX_PRIORITIES / 2,
        TickType_t timeout = portMAX_DELAY);

    static bool end(TickType_t timeout = portMAX_DELAY);
    static bool isReady();
    static void pause();
    static void resume();
    static bool redirect(StreamType type, StreamStorage stream);
    static bool redirect(Print& stream);
    static bool redirect(FILE* stream);

private:
    static mutex_t _mutex;
    static WorkerType _worker;
    static vprintf_like_t _previous_sink;
    static Log* _log;
    static size_t _log_index;
    static size_t _message_size;
    static TickType_t _timeout;
    static StreamType _stream_type;
    static StreamStorage _stream;

    static void _process(void*);
    static int _vprintf(const char* format, va_list args);
};
