all: bin/console-control bin/generic

bin/console-control: 
	$(MAKE) -C console-control

bin/generic:
	$(MAKE) -C generic

bin/sick:
	$(MAKE) -C sick

clean:
	$(MAKE) -C console-control clean
	$(MAKE) -C generic clean
