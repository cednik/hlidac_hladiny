#pragma once

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <mutex>
#include <functional>

class WorkerBase {
protected:
    WorkerBase():
        m_id { g_id++ }
    {}
    const int m_id;
    static const char* LOG_TAG;
private:
    static int g_id;
};

template <typename Tin, typename Tout = void, typename Tid = int, Tid END = -1, Tid PAUSE = -2>
class Worker: public WorkerBase
{
    //typedef std::recursive_mutex mutex_t;
    typedef Worker<Tin, Tout, Tid, END, PAUSE> This;
    typedef std::function<void(const Tid&, const Tin&, Tout*)> Process;

    struct Settings {
        int queue_size = 32;
        uint32_t stack_size = 2048;
        UBaseType_t task_priority = configMAX_PRIORITIES / 2;
    };
    struct Event {
        Tid type;
        TaskHandle_t task;
        UBaseType_t notification_index;
        Tin data;
        Tout* responce;
    };
public:
    Worker()
        : WorkerBase {},
          m_settings {},
          //m_mutex {},
          m_queue { nullptr },
          m_task { nullptr },
          m_paused { false },
          m_name { 0 }
    {
        snprintf(m_name, configMAX_TASK_NAME_LEN, "Worker%d", m_id);
    }
    ~Worker() {
        end();
    }

    This& queue_size(int size) {
        m_settings.queue_size = size;
        return *this;
    }
    int queue_size() { return m_settings.queue_size; }

    This& stack_size(uint32_t size) {
        m_settings.stack_size = size;
        return *this;
    }
    int stack_size() { return m_settings.stack_size; }

    This& task_priority(UBaseType_t priority) {
        m_settings.task_priority = priority;
        return *this;
    }
    UBaseType_t task_priority() { return m_settings.task_priority; }

    This& name(const char* _name) {
        strncpy(m_name, _name, configMAX_TASK_NAME_LEN-1);
        m_name[configMAX_TASK_NAME_LEN-1] = '\0';
        return *this;
    }
    const char* name() { return m_name; }

    This& process(const Process& fcn) {
        m_process = fcn;
        return *this;
    }

    bool isAlive() {
        return m_task;
    }

    bool isPaused() {
        return m_paused;
    }

    bool begin() {
        //std::lock_guard<mutex_t> lock (m_mutex);
        if (isAlive())
            end();
        m_queue = xQueueCreate(m_settings.queue_size, sizeof(Event));
        if (!m_queue) {
            ESP_LOGE(LOG_TAG, "Can not create %s event queue.", m_name);
            return false;
        }
        BaseType_t err = xTaskCreate(   _process,
                                        m_name,
                                        m_settings.stack_size,
                                        this,
                                        m_settings.task_priority,
                                        &m_task );
        if (err != pdPASS) {
            ESP_LOGE(LOG_TAG, "Error 0x%X occured while starting %s process task.", err, m_name);
            m_task = nullptr;
            vQueueDelete(m_queue);
            m_queue = nullptr;
            return false;
        }
        m_paused = false;
        return true;
    }
    bool end() {
        if (!isAlive())
            return true;
        resume();
        if (!query(END, Tin())) {
            ESP_LOGE(LOG_TAG, "Could not send END event to %s.", m_name);
            return false;
        }
        m_task = nullptr;
        return true;
    }

    bool pause() {
        if (!isAlive())
            return false;
        resume();
        if (!query(PAUSE, Tin())) {
            ESP_LOGE(LOG_TAG, "Could not send PAUSE event to %s.", m_name);
            return false;
        }
        m_paused = true;
        return true;
    }

    void halt() {
        if (!m_task)
            return;
        vTaskSuspend(m_task);
        m_paused = true;
    }

    void resume() {
        if (!m_task)
            return;
        vTaskResume(m_task);
        m_paused = false;
    }

    void kill() {
        if (m_task) {
            vTaskDelete(m_task);
            m_task = nullptr;
            vQueueDelete(m_queue);
            m_queue = nullptr;
        }
    }

    bool send(const Tid& event_type, const Tin& data, TickType_t timeout = portMAX_DELAY) {
        Event e {
            .type = event_type,
            .task = nullptr,
            .notification_index = 0,
            .data = data,
            .responce = nullptr
        };
        return xQueueSend(m_queue, &e, timeout) == pdTRUE;
    }
    bool query( const Tid& event_type, 
                const Tin& data,
                Tout* responce = nullptr,
                UBaseType_t notification_index = 0,
                TickType_t waitTimeout = portMAX_DELAY,
                TickType_t sendTimeout = portMAX_DELAY )
    {
        Event e {
            .type = event_type,
            .task = xTaskGetCurrentTaskHandle(),
            .notification_index = notification_index,
            .data = data,
            .responce = responce
        };
        if (xQueueSend(m_queue, &e, sendTimeout) != pdTRUE) {
            return false;
        }
        if (xTaskNotifyWaitIndexed(notification_index, 0, 0, nullptr, waitTimeout) != pdTRUE) {
            return false;
        }
        return true;
    }

private:
    static void _process(void* pWorker) {
        This& worker = *reinterpret_cast<This*>(pWorker);
        Event event;
        bool run = true;
        while(run) {
            if (xQueueReceive(worker.m_queue, &event, portMAX_DELAY)) {
                //std::lock_guard<mutex_t> lock (worker.m_mutex);
                switch (event.type) {
                case END:
                    run = false;
                    if (event.task != nullptr)
                        xTaskNotifyIndexed(event.task, event.notification_index, 0, eNoAction);
                    break;
                case PAUSE:
                    if (event.task != nullptr)
                        xTaskNotifyIndexed(event.task, event.notification_index, 0, eNoAction);
                    vTaskSuspend(nullptr);
                    break;
                default:
                    worker.m_process(event.type, event.data, event.responce);
                    if (event.task != nullptr)
                        xTaskNotifyIndexed(event.task, event.notification_index, 0, eNoAction);
                    break;
                }
            }
        }
        vTaskDelete(nullptr);
    }

    Settings m_settings;
    //mutex_t m_mutex;
    QueueHandle_t m_queue;
    TaskHandle_t m_task;
    bool m_paused;
    char m_name[configMAX_TASK_NAME_LEN];
    Process m_process;
};
