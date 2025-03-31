// Global Module Fragment : Optional
module;

#include "Common/Headers.h"

// Module Preamble : Required
export module CoroutineTask;

// Module Purview / Module Interface : Optional
export class CoroutineTask final
{
public:
    struct promise_type
    {
        CoroutineTask get_return_object() { return CoroutineTask{ this }; }

        std::suspend_always initial_suspend() { return { }; }
        std::suspend_always final_suspend() noexcept { return { }; }

        // https://github.com/naringcode/CPPStudyNotes/blob/main/Coroutines/coroutine_exceptions_exec_stages.cpp
        // 예외는 발생하지 않는다고 가정한다(자세한 건 직접 정리한 위 링크 참고).
        void unhandled_exception() {}

        void return_void() {}
    };

public:
    /*************************
    *      Rule of Five      *
    *************************/
    explicit CoroutineTask()
        : _handle{ }
    { }

    explicit CoroutineTask(promise_type* prom)
        : _handle{ std::coroutine_handle<promise_type>::from_promise(*prom) }
    { }

    ~CoroutineTask()
    {
        if (_handle != nullptr)
        {
            _handle.destroy();

            _handle = nullptr;
        }
    }

    CoroutineTask(const CoroutineTask& rhs) = delete; // copy constructor
    CoroutineTask& operator=(const CoroutineTask& rhs) = delete; // copy assignment

    // https://github.com/naringcode/CPPStudyNotes/blob/main/Coroutines/renew_coroutines_by_move.cpp
    // 이 부분은 정리한 문서 참고하기.
    CoroutineTask(CoroutineTask&& rhs) noexcept // move constructor
    {
        if (this == &rhs)
            return;

        // 도중에 코루틴을 해제할 수도 있기 때문에 코루틴 함수 내 자원은 RAII로 되어 있는 것이 좋다.
        if (_handle != nullptr)
        {
            _handle.destroy();

            _handle = nullptr;
        }

        std::swap(_handle, rhs._handle);
    }

    CoroutineTask& operator=(CoroutineTask&& rhs) noexcept // move assignment
    {
        if (this == &rhs)
            return *this;

        // 도중에 코루틴을 해제할 수도 있기 때문에 코루틴 함수 내 자원은 RAII로 되어 있는 것이 좋다.
        if (_handle != nullptr)
        {
            _handle.destroy();

            _handle = nullptr;
        }

        std::swap(_handle, rhs._handle);

        return *this;
    }

public:
    bool Done()
    {
        if (_handle == nullptr)
            return true;

        return _handle.done();
    }

    void Resume()
    {
        if (_handle.done() == false)
        {
            _handle.resume();
        }
    }

    void Destroy()
    {
        if (_handle != nullptr)
        {
            _handle.destroy();

            _handle = nullptr;
        }
    }

private:
    std::coroutine_handle<promise_type> _handle = nullptr;
};

// Private Module Fragment : Optional
module: private;
