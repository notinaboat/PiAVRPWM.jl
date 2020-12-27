using PiGPIOMEM
using BBSPI

BBSPI.delay(s::BBSPI.SPISlave) = PiGPIOMEM.spin(1000)

function go()

    spi = BBSPI.SPISlave(  cs = Ref(false),
                          clk = GPIOPin(25; output=true),
                         mosi = GPIOPin(10; output=true),
                         miso = Ref(false));
    stop = Ref(false)

    @async begin
        readavailable(stdin)
        stop[] = true
    end
    
    while !stop[]
        println("sending...")
        BBSPI.transfer(spi, UInt8[0])
        sleep(0.01)
        BBSPI.transfer(spi, UInt8[0b01000000])
        sleep(0.01)
        BBSPI.transfer(spi, UInt8[0b10010000])
        sleep(2)
    end
end
