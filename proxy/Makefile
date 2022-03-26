CC := gcc
CCFLAGS := -g

TARGET := proxy

all: $(TARGET)

%: %.c
	$(CC) $(CCFLAGS) $< -o $@

clean:
	rm -f $(TARGET)
