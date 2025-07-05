FROM my-picar-autonomous:latest

# Install system dependencies
RUN apt update && apt install -y portaudio19-dev python3-dev

# Copy and install robot libraries
COPY robot-hat/ /tmp/robot-hat/
COPY picar-x/ /tmp/picar-x/

# Install libraries properly
RUN cd /tmp/robot-hat && pip3 install .
RUN cd /tmp/picar-x && pip3 install .

# Apply picarx patch
RUN python3 -c "import picarx, os; picarx_file = picarx.__file__.replace('__init__.py', 'picarx.py'); content = open(picarx_file).read(); content = content.replace('os.getlogin()', '__import__(\"getpass\").getuser()'); open(picarx_file, 'w').write(content)"

# Test installation
RUN python3 -c "from picarx import Picarx; print('Libraries installed successfully')"
