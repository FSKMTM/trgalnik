CXX=g++
trgalnik: trgalnik.cpp
	$(CXX) trgalnik.cpp -lpigpio -lrt -lpthread -o trgalnik
