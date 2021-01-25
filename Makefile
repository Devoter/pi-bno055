C=gcc
CFLAGS= -O3 -Wall -g
LIBS= -lm
AR=ar

ALLBIN=getbno055 libbno055.so

all: ${ALLBIN}

clean:
	rm -f *.o ${ALLBIN}

libbno055.o:
	${CC} -c libbno055.c -fPIC

getbno055: print.o getbno055.o libbno055.o
	$(CC) libbno055.o print.o getbno055.o -o getbno055 ${LIBS}

libbno055.so: libbno055.o
	$(CC) libbno055.o libbno055.h -shared -o libbno055.so ${LIBS}
