CC = g++  
BIN = demo

CFLAGS += -std=c++11 -Wall -g -fPIC -Iheaders  
BUILD_DIR = build

SRCS = $(wildcard srcs/*.c)  
HEDS = $(wildcard headers/*.h)  
OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(notdir $(SRCS)))  
DEPS = $(patsubst %.o, %.d, $(OBJS))

vpath %.h headers
vpath %.c srcs

.PHONY: all
all: $(BIN)
	@echo "Start compiling"

-include $(DEPS)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $(BIN) $^

$(OBJS): | _PRE

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -MMD -c -o $@ $<

.PHONY: _PRE
_PRE:  
	mkdir -p $(BUILD_DIR)

.PHONY: clean
clean:  
	rm -fr build

