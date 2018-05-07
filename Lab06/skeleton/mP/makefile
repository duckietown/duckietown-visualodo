CC= gcc
CC2= g++

CFLAGS=`pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`

TARGET= lab06_test
OBJECTS= pid.o lab06_test.o tracking.o

all: clean compile link

#NEW RULES MORE VERSATILE (CAN CHANGE FILES NAMES)
compile: $(OBJECTS)

link: $(TARGET).out

clean:
	rm -f *.o
	rm -f *.out

.PHONY: clean all compile link

# IMPLICIT RULE FOR COMPILING OBJECTS
%.o: %.c
	$(CC2) -c $^ -o $@ -Wall $(CFLAGS)
	
%.out:
	$(CC2) -o $(TARGET).out $(OBJECTS) -Wall $(LDFLAGS)

#OLD RULES
#compile: pid.h
#	$(CC2) -c pid.c -o pid.o -Wall $(CFLAGS)
#	$(CC2) -c lab06_test.c -o lab06_test.o -Wall $(CFLAGS)

#link: pid.h
#	$(CC2) -o lab06_test.out pid.o lab06_test.o -Wall $(LDFLAGS)

