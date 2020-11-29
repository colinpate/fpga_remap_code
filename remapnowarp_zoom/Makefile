TARGET = houghlines

CFLAGS = -Ofast -Wall -std=c++11 `pkg-config --cflags opencv`
LDFLAGS = -Ofast -Wall -std=c++11 `pkg-config --libs opencv`
CC = g++

all: $(TARGET)

$(TARGET): $(TARGET).o
	$(CC) $(LDFLAGS) -o $@ $^ `pkg-config --libs opencv` -lpthread

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) *.a *.o *~

