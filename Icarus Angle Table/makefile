angleTable: angleTable.o angleTablePreprocessing.o
	${CC} -o angleTable angleTable.o angleTablePreprocessing.o
angleTable.o: angleTable.c
	${CC} -c angleTable.c
data.o: angleTablePreprocessing.c
	${CC} -c angleTablePreprocessing.c
clean:
	${RM} angleTable *.o
