build-rv1106-bins: build-rv1106-serial-flash
	chmod +x rv1106/fs/usr/bin/*

build-rv1106-serial-flash:
	cd rv1106/serial-flash; \
	env GOOS=linux GOARCH=arm go build -o serial-flash -ldflags="-s -w";
	cp rv1106/serial-flash/serial-flash rv1106/fs/usr/bin/serial-flash

update-rv1106-bins: build-rv1106-bins
	scp -rp rv1106/fs/usr rv1106:/