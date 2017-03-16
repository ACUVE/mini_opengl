set -ve

OUTDIRECTORY="linux"
CXXFLAGS="-Wall -O3 -std=c++1z"
# LIBS="-lOpenGL -lGLEW -lglfw -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d -pthread"
LIBS="-lOpenGL -lGLEW -lglfw -pthread"

cd `dirname $0`

mkdir -p "${OUTDIRECTORY}"
cd "${OUTDIRECTORY}"

for f in ../*.cpp
do
    g++ ${CXXFLAGS} -Wall -c "$f"
done

g++ ${LIBS} *.o -o DCM
