all: bin/console-control


bin/console-control: 
	$(MAKE) -C console-control


clean:
	$(MAKE) -C console-control clean
