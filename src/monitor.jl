using Sockets
using Serialization
using IterTools
using Printf

using PiGPIOC: gpioCfgClock, gpioSetMode, gpioWrite
using PiGPIOC: gpioSerialReadOpen, gpioSerialReadClose
using PiGPIOC: gpioInitialise, gpioDelay, gpioSetMode, gpioSerialRead
using PiGPIOC: PI_INIT_FAILED, PI_INPUT, PI_OUTPUT

const RESET_PIN = 8

const SERIAL_PIN = 9
const SERIAL_BAUD = 10000

buffer = Channel{UInt32}(10000)

# Configure pigpio...
gpioCfgClock(10, 1, 1); # us, PCM, ignored
res = gpioInitialise();
@assert(res != PI_INIT_FAILED)

const TAG_ERROR  = 0b111
const TAG_PRINT  = 0b000
const TAG_PRINTN = 0b100

const linecache = Dict{Int, Tuple{String,String}}()

function go()
# Hold AVR in RESET...
gpioSetMode(RESET_PIN, PI_OUTPUT)
gpioWrite(RESET_PIN, 0)

# Start reading from serial port...
err = gpioSerialReadOpen(SERIAL_PIN, SERIAL_BAUD, 32);
@assert(err == 0);

# Release AVR RESET...
gpioDelay(100000)#us
gpioWrite(RESET_PIN, 1)
gpioSetMode(RESET_PIN, PI_INPUT)

stop = Ref(false)

@async begin
    readavailable(stdin)
    stop[] = true
end

    println("go...")
while !stop[]

    v = [UInt32(0)]

    n = gpioSerialRead(SERIAL_PIN, v, sizeof(UInt32));
    @assert(n == 0 || n == sizeof(UInt32))

    if n == sizeof(UInt32)
        tag = v[1] >> 29
        if tag in (TAG_ERROR, TAG_PRINT, TAG_PRINTN)
            address = if tag == TAG_PRINTN
                ((v[1] & 0x7FFF0000) >> 16) * 2
            else
                (v[1] & 0xFFFF) * 2
            end
            #@show address
            if haskey(linecache, address)
                source_line, line = linecache[address]
            else
                cmd = `avr-addr2line -e main.elf 0x$(string(address, base=16))`
                source_line = try chomp(read(cmd, String)) catch err "?" end
                file, line_no = split(source_line, ":")
                line = try
                    nth(eachline(file), parse(Int, line_no))
                catch err
                    string(v[1], base=16)
                end
                linecache[address] = (source_line, line)
            end
            if tag == TAG_ERROR
                println("ERROR: ", line, " ", source_line, " ", address)
            elseif tag == TAG_PRINT
                m = match(r"\s*println\(\"([^\"]*)\".*", line)
                if m == nothing
                    println(line)
                else
                    println(m[1])
                end
            elseif tag == TAG_PRINTN
                m = match(r"\s*printn\(\"([^\"]*)\".*", line)
                if m == nothing
                    println(line)
                else
                    println(m[1], v[1] & 0xFFFF)
                end
            end
        else
            @show v[1]
        end
    else
        sleep(0.005)
    end
end

gpioSerialReadClose(SERIAL_PIN)
end
