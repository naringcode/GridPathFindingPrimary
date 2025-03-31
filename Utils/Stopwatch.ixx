// Global Module Fragment : Optional
module;

#include "Common/Headers.h"

// Module Preamble : Required
export module Stopwatch;

// Module Purview / Module Interface : Optional
export class Stopwatch final
{
private:
    using ClockT  = std::chrono::steady_clock;
    using SecondT = std::chrono::duration<f64>;
    using MilliT  = std::chrono::duration<f64, std::milli>;
    using MicroT  = std::chrono::duration<f64, std::micro>;
    using NanoT   = std::chrono::duration<f64, std::nano>;
    
    enum class TimePoint
    {
        // 시간 간격 예시
        // 
        // -> 흐름 방향 ->
        // |----------------|
        // ^                ^
        // Previous       Latest
        // 
        // Split() 호출
        // |----------------|----------------|
        //                  ^                ^
        //               Previous          Latest
        Latest,
        Previous,

        Max
    };

public:
    /*************************
    *      Rule of Five      *
    *************************/
    explicit Stopwatch() = default; // constructor
    ~Stopwatch() = default; // destructor

    Stopwatch(const Stopwatch& rhs) = delete; // copy constructor
    Stopwatch& operator=(const Stopwatch& rhs) = delete; // copy assignment

    Stopwatch(Stopwatch&& rhs) noexcept = delete; // move constructor
    Stopwatch& operator=(Stopwatch&& rhs) noexcept = delete; // move assignment

public:
    void Start();
    void Stop();

    void Reset();
    void CaptureLapTime();
    
public:
    // Total Elapsed Time
    f64 GetTotalElapsedSeconds() const;
    f64 GetTotalElapsedMilli() const;
    f64 GetTotalElapsedMicro() const;
    f64 GetTotalElapsedNano() const;

    // Lap Time
    f64 GetLapTimeSeconds() const;
    f64 GetLapTimeMilli() const;
    f64 GetLapTimeMicro() const;
    f64 GetLapTimeNano() const;
    
private:
    std::chrono::time_point<ClockT> _startTP;
    std::chrono::time_point<ClockT> _splitTPs[(i32)TimePoint::Max];
};

// Private Module Fragment : Optional
module: private;

void Stopwatch::Start()
{
    _startTP = ClockT::now();

    _splitTPs[(i32)TimePoint::Latest]   = _startTP;
    _splitTPs[(i32)TimePoint::Previous] = _startTP;
}

void Stopwatch::Stop()
{
    _startTP = ClockT::now();

    _splitTPs[(i32)TimePoint::Latest]   = _startTP;
    _splitTPs[(i32)TimePoint::Previous] = _startTP;
}

void Stopwatch::Reset()
{
    _startTP = ClockT::now();

    _splitTPs[(i32)TimePoint::Latest]   = _startTP;
    _splitTPs[(i32)TimePoint::Previous] = _startTP;
}

void Stopwatch::CaptureLapTime()
{
    _splitTPs[(i32)TimePoint::Previous] = _splitTPs[(i32)TimePoint::Latest];
    _splitTPs[(i32)TimePoint::Latest]   = ClockT::now();
}

f64 Stopwatch::GetTotalElapsedSeconds() const
{
    auto endTP = ClockT::now();

    return std::chrono::duration_cast<SecondT>(endTP - _startTP).count();
}

f64 Stopwatch::GetTotalElapsedMilli() const
{
    auto endTP = ClockT::now();

    return std::chrono::duration_cast<MilliT>(endTP - _startTP).count();
}

f64 Stopwatch::GetTotalElapsedMicro() const
{
    auto endTP = ClockT::now();

    return std::chrono::duration_cast<MicroT>(endTP - _startTP).count();
}

f64 Stopwatch::GetTotalElapsedNano() const
{
    auto endTP = ClockT::now();

    return std::chrono::duration_cast<NanoT>(endTP - _startTP).count();
}

f64 Stopwatch::GetLapTimeSeconds() const
{
    auto interval = _splitTPs[(i32)TimePoint::Latest] - _splitTPs[(i32)TimePoint::Previous];

    return std::chrono::duration_cast<SecondT>(interval).count();
}

f64 Stopwatch::GetLapTimeMilli() const
{
    auto interval = _splitTPs[(i32)TimePoint::Latest] - _splitTPs[(i32)TimePoint::Previous];

    return std::chrono::duration_cast<MilliT>(interval).count();
}

f64 Stopwatch::GetLapTimeMicro() const
{
    auto interval = _splitTPs[(i32)TimePoint::Latest] - _splitTPs[(i32)TimePoint::Previous];

    return std::chrono::duration_cast<MicroT>(interval).count();
}

f64 Stopwatch::GetLapTimeNano() const
{
    auto interval = _splitTPs[(i32)TimePoint::Latest] - _splitTPs[(i32)TimePoint::Previous];

    return std::chrono::duration_cast<NanoT>(interval).count();
}
