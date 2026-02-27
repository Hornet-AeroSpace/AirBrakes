CC = gcc
CFLAGS = -O0 -mavx2

filter: filter.c
	gcc -O0 filter.c matrix_functions.c -o filter -mavx2 -lm
	
clean:
	rm -f *o. filter