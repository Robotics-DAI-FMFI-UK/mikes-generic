all: bin/console-control bin/generic bin/sick bin/localize_rect

bin/console-control:
	$(MAKE) -C console-control

bin/generic:
	$(MAKE) -C generic

bin/sick:
	$(MAKE) -C sick

bin/localize_rect:
	$(MAKE) -C localize_rect

clean:
	$(MAKE) -C console-control clean
	$(MAKE) -C sick clean
	$(MAKE) -C generic clean
	$(MAKE) -C localize_rect clean
