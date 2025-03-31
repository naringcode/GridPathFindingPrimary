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
        // ���ܴ� �߻����� �ʴ´ٰ� �����Ѵ�(�ڼ��� �� ���� ������ �� ��ũ ����).
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
    // �� �κ��� ������ ���� �����ϱ�.
    CoroutineTask(CoroutineTask&& rhs) noexcept // move constructor
    {
        if (this == &rhs)
            return;

        // ���߿� �ڷ�ƾ�� ������ ���� �ֱ� ������ �ڷ�ƾ �Լ� �� �ڿ��� RAII�� �Ǿ� �ִ� ���� ����.
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

        // ���߿� �ڷ�ƾ�� ������ ���� �ֱ� ������ �ڷ�ƾ �Լ� �� �ڿ��� RAII�� �Ǿ� �ִ� ���� ����.
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
