# Makefile for SDR project

# -- Directories --
IDIR = ./inc
LDIR = ./lib
SDIR = ./src
ODIR = ./obj
BDIR = ./bin

# -- Compiler and flags --
CC = CL
#CFLAGS = /O2 /EHsc /I$(IDIR) /MP  /Dkiss_fft_scalar=int16_t /DFIXED_POINT=16
CFLAGS = /Zi /O2 /EHsc /I$(IDIR) /MP  /Dkiss_fft_scalar=int16_t /DFIXED_POINT=16
LDFLAGS = /link /DEBUG
#LDFLAGS = /link

# -- Metadata --
CURRENT_HASH := '"$(shell git rev-parse HEAD)"'
CURRENT_DATE := '"$(shell date /t)"'

# -- Version Information --
sdr collector relay_server:
sdr: MAJOR=0
sdr: MINOR=1
sdr: PATCH=1	
sdr: APP_NAME='"sdr"'

collector: MAJOR=0
collector: MINOR=1
collector: PATCH=0
collector: APP_NAME='"collector"'

relay_server: MAJOR=0
relay_server: MINOR=1
relay_server: PATCH=0
relay_server: APP_NAME='"relay_server"'

MACROS = /DMAJOR_VERSION=$(MAJOR) /DMINOR_VERSION=$(MINOR) \
	/DPATCH_VERSION=$(PATCH) /DAPP_NAME=$(APP_NAME) \
	/DCURRENT_HASH=$(CURRENT_HASH) /DCURRENT_DATE=$(CURRENT_DATE)

SDR_OBJS = $(ODIR)/sdr.obj \
           $(ODIR)/ElasticReceiver.obj \
           $(ODIR)/NCO.obj \
           $(ODIR)/ChannelProcessor.obj \
           $(ODIR)/g2init.obj \
           $(ODIR)/PCSEngine.obj \
           $(ODIR)/kiss_fft.obj

# -- Targets --
all: setup sdr collector relay_server

setup:
	@if not exist $(ODIR) mkdir $(ODIR)
	@if not exist $(BDIR) mkdir $(BDIR)

# Pattern rule: ALL .cpp to .obj
$(ODIR)/%.obj: $(SDIR)/%.cpp
	$(CC) $(CFLAGS) $(MACROS) /c $< /Fo$@

# Rule for the C file (kiss_fft.c)
$(ODIR)/kiss_fft.obj: $(SDIR)/kiss_fft.c
	$(CC) $(CFLAGS) $(MACROS) /c $< /Fo$@

# Build rules for each target
sdr: setup $(SDR_OBJS)
	$(CC) $(SDR_OBJS) $(LDFLAGS) ws2_32.lib /OUT:$(BDIR)/sdr.exe

collector: setup $(ODIR)/collector.obj
	$(CC) $(ODIR)/collector.obj $(LDFLAGS) ws2_32.lib $(LDIR)/FTD2XX.lib \
	/OUT:$(BDIR)/collector.exe

relay_server: setup $(ODIR)/relay_server.obj
	$(CC) $(ODIR)/relay_server.obj $(LDFLAGS) ws2_32.lib \
	/OUT:$(BDIR)/relay_server.exe

clean:
	@if exist $(BDIR) rm -f $(BDIR)/*
	@if exist $(ODIR) rm -f $(ODIR)/*
	@if exist *.obj rm -f *.obj
	@echo Environment Cleaned.