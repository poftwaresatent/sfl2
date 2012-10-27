AM_CPPFLAGS= -I@abs_top_srcdir@/3rdparty/fpplib/include \
             -I@abs_top_srcdir@/3rdparty/fpplib/yaml-cpp-0.2.7/include \
             @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

noinst_LTLIBRARIES=   libnpm3p.la
libnpm3p_la_SOURCES=  fpplib/src/configurable.cpp \
                      fpplib/src/factory.cpp \
                      fpplib/src/parameter.cpp \
                      fpplib/src/yaml_parser.cpp \
                      fpplib/yaml-cpp-0.2.7/src/aliasmanager.cpp \
                      fpplib/yaml-cpp-0.2.7/src/contrib/graphbuilder.cpp \
                      fpplib/yaml-cpp-0.2.7/src/contrib/graphbuilderadapter.cpp \
                      fpplib/yaml-cpp-0.2.7/src/conversion.cpp \
                      fpplib/yaml-cpp-0.2.7/src/directives.cpp \
                      fpplib/yaml-cpp-0.2.7/src/emitfromevents.cpp \
                      fpplib/yaml-cpp-0.2.7/src/emitter.cpp \
                      fpplib/yaml-cpp-0.2.7/src/emitterstate.cpp \
                      fpplib/yaml-cpp-0.2.7/src/emitterutils.cpp \
                      fpplib/yaml-cpp-0.2.7/src/exp.cpp \
                      fpplib/yaml-cpp-0.2.7/src/iterator.cpp \
                      fpplib/yaml-cpp-0.2.7/src/node.cpp \
                      fpplib/yaml-cpp-0.2.7/src/nodebuilder.cpp \
                      fpplib/yaml-cpp-0.2.7/src/nodeownership.cpp \
                      fpplib/yaml-cpp-0.2.7/src/null.cpp \
                      fpplib/yaml-cpp-0.2.7/src/ostream.cpp \
                      fpplib/yaml-cpp-0.2.7/src/parser.cpp \
                      fpplib/yaml-cpp-0.2.7/src/regex.cpp \
                      fpplib/yaml-cpp-0.2.7/src/scanner.cpp \
                      fpplib/yaml-cpp-0.2.7/src/scanscalar.cpp \
                      fpplib/yaml-cpp-0.2.7/src/scantag.cpp \
                      fpplib/yaml-cpp-0.2.7/src/scantoken.cpp \
                      fpplib/yaml-cpp-0.2.7/src/simplekey.cpp \
                      fpplib/yaml-cpp-0.2.7/src/singledocparser.cpp \
                      fpplib/yaml-cpp-0.2.7/src/stream.cpp \
                      fpplib/yaml-cpp-0.2.7/src/tag.cpp \
                      fpplib/yaml-cpp-0.2.7/util/parse.cpp
