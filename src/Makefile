FLAGS = -std=c99 -Wall
PIGPIO = -lpigpio -lrt -lm
SOURCES = pid.c gyro.c client.c
OBJECTS = pid.o gyro.o client.o
HEADERS = pid.h gyro.h
EXEBIN = client

all: $(EXEBIN)

$(EXEBIN) : $(OBJECTS) $(HEADERS)
	gcc -o $(EXEBIN) $(OBJECTS) $(PIGPIO)

$(OBJECTS) : $(SOURCES) $(HEADERS)
	gcc -c $(FLAGS) $(SOURCES) $(PIGPIO)

clean :
	find . -exec touch {} \;
	rm -f $(EXEBIN) $(OBJECTS)
