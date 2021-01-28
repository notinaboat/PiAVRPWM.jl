"""
# PiAVRPWM.jl

17 Channel PWM output using ATMega328p.
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
        new(AVRDevice(;c_code = joinpath(@__DIR__, "main.c"), args...))
end

function set_pwm_level(pwm, channel, level)
    @assert level >= 0.0 && level <= 1.0
    l = round(UInt16, level * pwm_max)
    write(pwm.avr, UInt8[channel,
                         0b01000000 | l >> 6,
                         0b10000000 | l & 0b00111111])
end

Base.setindex!(pwm::AVRPWM, level, channel) = set_pwm_level(pwm, channel, level)

end # module
