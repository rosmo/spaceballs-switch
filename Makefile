all: fmt

.PHONY: fmt

SOURCES := main/main.c main/main.h

fmt: 
	astyle \
		-n \
		--style=otbs \
		--attach-namespaces \
		--attach-classes \
		--indent=spaces=4 \
		--convert-tabs \
		--align-reference=name \
		--keep-one-line-statements \
		--pad-header \
		--pad-oper \
		--unpad-paren \
		--max-continuation-indent=120 \
		$(SOURCES)
