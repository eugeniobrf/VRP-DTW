# Variaveis
CC = g++
CFLAGS = -m64 -g -O3
ARQUIVO = main
INCLUDES = -I ~/gurobi912/linux64/include/ -L ~/gurobi912/linux64/lib -l gurobi_c++ -l gurobi91 -lm
ifndef FILE
FILE = ./instances/R202_100.csv
endif
ifndef SEED
SEED = 123
endif

all: $(ARQUIVO)
choices.o: choices.h
	$(CC) $(CFLAGS) -c choices.h -o choices.o
route.o: route.h	
	$(CC) $(CFLAGS) -c route.h -o route.o
solution.o: solution.h	
	$(CC) $(CFLAGS) -c solution.h -o solution.o
utils.o: utils.h	
	$(CC) $(CFLAGS) -c utils.h -o utils.o
vertex.o: vertex.h	
	$(CC) $(CFLAGS) -c vertex.h -o vertex.o
$(ARQUIVO).o: choices.o route.o solution.o utils.o vertex.o $(ARQUIVO).cpp
	$(CC) $(CFLAGS) -c $(ARQUIVO).cpp -o $(ARQUIVO).o $(INCLUDES)
$(ARQUIVO): $(ARQUIVO).o
	$(CC) $(CFLAGS) -o $(ARQUIVO) $(ARQUIVO).o $(INCLUDES)
clean: 
	rm -rf *.o $(ARQUIVO)
run: $(ARQUIVO)
	./main $(FILE) $(SEED)