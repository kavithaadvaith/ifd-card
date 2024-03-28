CC = arm-none-linux-gnueabi-gcc
CFLAGS = -Iinclude

SRCDIR = src
OBJDIR = obj
LIB_DIR = lib
EXE_DIR = exe

SRC = $(wildcard $(SRCDIR)/*.c)
OBJ = $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(SRC))

#LIBS = -lgl11pos

LDFLAG = /opt/freescale/usr/local/gcc-4.1.2-glibc-2.5-nptl-3/arm-none-linux-gnueabi/arm-none-linux-gnueabi/sysroot/usr/include/

#LIBS = -L ./lib/lgl11pos

TARGET = exe/Card_Test_Tool

all: $(TARGET)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS)  $^ -o $@ 
#	$(CC) $(CFLAGS)  $^ -o $@ $(LIBS) 
	#$(CC) $(CFLAGS) -I $(LDFLAG)  $^ -o $@ $(LIBS) 

clean:
	rm -f $(OBJDIR)/*.o $(TARGET)
