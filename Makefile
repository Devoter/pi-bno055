C=gcc
CFLAGS= -O3 -Wall -g
LIBS= -lm
AR=ar

ALLBIN=getbno055 libbno055.so

all: ${ALLBIN}

clean:
	rm -f *.o ${ALLBIN}

getbno055: i2c_bno055.o getbno055.o
	$(CC) i2c_bno055.o getbno055.o -o getbno055 ${LIBS}

i2c_bno055.o:
	${CC} -c i2c_bno055.c -fPIC

libbno055.so: i2c_bno055.o
	$(CC) i2c_bno055.o getbno055.h -shared -o libbno055.so ${LIBS}
