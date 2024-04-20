deps_config := \
	src/modules/src/Kconfig \
	src/deck/drivers/src/Kconfig \
	app_api/Kconfig \
	Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
