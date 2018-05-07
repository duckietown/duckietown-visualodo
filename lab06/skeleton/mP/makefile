CC= gcc
CC2= g++

CFLAGS=`pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`

compile: pid.h
	$(CC2) -c pid.c -o pid.o -Wall $(CFLAGS)
	$(CC2) -c lab06_test.c -o lab06_test.o -Wall $(CFLAGS)

link: pid.h
	$(CC2) -o pid.out pid.o lab06_test.o -Wall $(LDFLAGS)
	
all: compile link

clean:
	-rm *.o
	-rm *.out
