if BUILD_BIRON
  BIRON_DIR=biron
endif
SUBDIRS=       common robox visitor laas $(BIRON_DIR) simul
BUILT_SOURCES= npm

npm:
	test -L npm || ln -s $(top_srcdir) npm
