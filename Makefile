PREFIX ?= ${HOME}/local/DIR/vispy

install:
	mkdir -p $(PREFIX)/lib/python/site-packages/vispy/examples
	install engine.py $(PREFIX)/lib/python/site-packages/vispy
	install objects_lib.py $(PREFIX)/lib/python/site-packages/vispy
	install __init__.py $(PREFIX)/lib/python/site-packages/vispy
	cp -r examples/* $(PREFIX)/lib/python/site-packages/vispy/examples

sys-dep:
	sudo apt-get install python-opengl python-pygame python-numpy
