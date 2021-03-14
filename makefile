
BINDIR = bin


all: | $(BINDIR)
	g++ -std=c++17 -Wall -Wextra -o $(BINDIR)/corridor environments/corridor.cpp

$(BINDIR):
	mkdir -p bin
