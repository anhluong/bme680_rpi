INC=-IBME680_driver -IBME680_driver/Self\ test
CFLAGS+= -Wall -I$(INC) -std=c99 -D_XOPEN_SOURCE=500 -O2 $(ARM_OPTIONS)
LDFLAGS+=-lpthread -lm #-lwiringPi

bme680-objs := BME680_driver/bme680.o

all: clean main

clean:
	rm -f clean main *.o BME680_driver/*.o

main: main.o $(bme680-objs)
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)
