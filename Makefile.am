#SUBDIRS=       common robox visitor laas simul
SUBDIRS=       common simul
#dist_doc_DATA= LICENSE.GPL README

#BUILT_SOURCES= npm subdirs
BUILT_SOURCES= npm

npm:
	test -L npm || ln -s $(top_srcdir) npm

subdirs:
	mkdir -p $(SUBDIRS)
	echo "subdirs created `date`" > subdirs
