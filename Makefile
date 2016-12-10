
all: ninjasitl 

ninjasitl:
	mkdir -p build; cd build; cmake ../src && make && cp quadcopter ../

docs:
	doxygen scripts/doxygen

run: ninjasitl
	./start-quadsim.sh

.PHONY: docs ninjasitl run
