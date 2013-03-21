PREFIX ?= ${HOME}/local/DIR/avispy

install:
	mkdir -p $(PREFIX)/lib/python/site-packages/avispy/examples
	install engine.py $(PREFIX)/lib/python/site-packages/avispy
	install objects_lib.py $(PREFIX)/lib/python/site-packages/avispy
	install __init__.py $(PREFIX)/lib/python/site-packages/avispy
	cp -r examples/* $(PREFIX)/lib/python/site-packages/avispy/examples

sys-dep:
	sudo apt-get install python-opengl python-pygame python-numpy
