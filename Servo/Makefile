CC=aarch64-none-linux-gnu-gcc
CFLAGS=-Wall -O2
LDFLAGS=-lm
OBJS=main.o fifo_rx.o tracker.o pca9685.o

all: servo_track

servo_track: $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) servo_track
