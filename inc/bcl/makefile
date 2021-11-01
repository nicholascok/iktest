CC			:= gcc
CC_FLAGS	:= -lm -O0

TARGET_DIR	:= build
HEADERS		:= 
OBJS		:= bmap.c

.PHONY: all build clean

all: build

build:
	@echo "building..."
	@mkdir -p "./build/"
	$(CC) -o ./$(TARGET_DIR)/bmap $(HEADERS) $(OBJS) $(CC_FLAGS)
	@echo -e "\ndone."

clean:
	@echo "cleaning build directory..."
	@rm -r ./$(TARGET_DIR)/ 2>/dev/null && echo -e "\ndone." || echo -e "\nalready clean."
