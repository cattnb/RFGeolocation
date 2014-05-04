TOOLS_TARGETS	:= my2cset

all: Client

Client: Client.o i2cbusses.o
	$(CC) $(LDFLAGS) -std=c99 -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) $(TOOLS_CFLAGS) -std=c99 -c $< -o $@

clean:
	rm i2cbusses.o Client Client.o
