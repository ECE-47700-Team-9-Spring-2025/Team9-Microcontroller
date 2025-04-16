follow-output:
	screen $$(ls /dev/tty.usbserial*) 115200

follow-output-debug:
	@echo "Checking for USB to TTL devices..."
	@echo "Available serial devices:"
	@ls /dev/tty.* /dev/cu.* 2>/dev/null || echo "No serial devices found"
	@echo ""
	@echo "Looking specifically for FTDI devices..."
	@ls /dev/tty.usbserial* /dev/cu.usbserial* 2>/dev/null || echo "No FTDI devices found"
	@echo ""
	@if [ -z "$$(ls /dev/tty.usbserial* /dev/cu.usbserial* 2>/dev/null)" ]; then \
		echo "Troubleshooting steps:"; \
		echo "1. Make sure your FT232RL adapter is properly connected"; \
		echo "2. Check if you need to install FTDI drivers"; \
		echo "3. Try unplugging and reconnecting the device"; \
		echo ""; \
		echo "Would you like to proceed with manual device selection? (y/n)"; \
		read answer; \
		if [ "$$answer" = "y" ]; then \
			echo "Available devices:"; \
			ls /dev/tty.* /dev/cu.* 2>/dev/null; \
			echo ""; \
			read -p "Enter device path: " device; \
			if [ -n "$$device" ]; then \
				screen $$device 115200; \
			else \
				echo "No device selected. Exiting."; \
				exit 1; \
			fi; \
		else \
			echo "Exiting."; \
			exit 1; \
		fi; \
	else \
		device=$$(ls /dev/cu.usbserial* 2>/dev/null | head -1); \
		echo "Auto-selected: $$device"; \
		screen $$device 115200; \
	fi