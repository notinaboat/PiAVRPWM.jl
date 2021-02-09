"""
# PiAVRPWM.jl

Multi-channel 0-5V soft PWM (PDM) Output Module using ATMega328p.
"""
module PiAVRPWM

export AVRPWM

using PiAVR

const pwm_max = begin
    main_c = joinpath(@__DIR__, "main.c")
    max = read(`awk '/#define PWM_MAX/ {print$3}' $main_c`, String) |> chomp
    parse(Float64, max)
end

struct AVRPWM
    avr::AVRDevice
    AVRPWM(; args...) =
        new(AVRDevice(;c_file = joinpath(@__DIR__, "main.c"), args...))
end

function set_pwm_level(pwm, channel, level)
    @assert level >= 0.0 && level <= 1.0
    l = round(UInt16, level * pwm_max)
    write(pwm.avr, UInt8[channel << 3 | l >> 7, 0b10000000 | (l & 0b01111111)])
end

Base.setindex!(pwm::AVRPWM, level, channel) = set_pwm_level(pwm, channel, level)

end # module
