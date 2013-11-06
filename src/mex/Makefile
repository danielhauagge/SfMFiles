all: sfmf_load_bundle.$(shell mexext)

%.$(shell mexext): %.cpp
	mex -g `pkg-config --libs --cflags sfmfiles eigen3 cmdcore` -lboost_system $<

install: sfmf_load_bundle.$(shell mexext)
	cp $< ~/Documents/MATLAB/