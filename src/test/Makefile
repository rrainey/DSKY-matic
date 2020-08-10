prefix = /usr/local

all: alarm-panel-test

alarm-panel-test: alarm-panel-test.c
	@echo "CFLAGS=$(CFLAGS)" | \
                fold -s -w 70 | \
                sed -e 's/^/# /'
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDCFLAGS) -o $@ $^

install: alarm-panel-test
	install -D alarm-panel-test \
                $(DESTDIR)$(prefix)/bin/alarm-panel-test

clean:
	-rm -f alarm-panel-test *.o

distclean: clean

uninstall:
	-rm -f $(DESTDIR)$(prefix)/bin/alarm-panel-test

.PHONY: all install clean distclean uninstall
