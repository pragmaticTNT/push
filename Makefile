# ===> Linux 
# NOTE: add the path to the pkgconfig folder of glfw3 you have install to the PKG_CONFIG_PATH as follows:
# 	export PKG_CONFIG_PATH=$(PKG_CONFIG_PATH:<path to pkgconfig folder>
# currently the path is: /home/xyl9/glfw_ws/glfw-3.2/install/lib/pkgconfig

CCFLAGS = -std=c++11 -g -O3 `pkg-config --cflags glfw3 box2d`
LDFLAGS = `pkg-config --static --libs glfw3 box2d` -lGL

# ===> OS 
# CCFLAGS = -g -O3 -I /usr/local/include -framework OpenGL
# LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

SRC = main.cc push.cc gui.cc
HDR = push.hh

all: push

push: $(SRC) $(HDR)
	g++ $(CCFLAGS) $(SRC) $(LDFLAGS) -o $@ 

clean:
	rm -f push 
	rm -f *.o
