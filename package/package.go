package rdno_apps

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_sensors "github.com/jurgen-kluft/rdno_sensors/package"
	rdno_u8g2 "github.com/jurgen-kluft/rdno_u8g2/package"
	rdno_wifi "github.com/jurgen-kluft/rdno_wifi/package"
)

const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rdno_apps"
)

func GetPackage() *denv.Package {
	wifipkg := rdno_wifi.GetPackage()
	sensorspkg := rdno_sensors.GetPackage()
	u8g2pkg := rdno_u8g2.GetPackage()

	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(wifipkg)
	mainpkg.AddPackage(sensorspkg)

	// Setup the main applications
	airquality := denv.SetupCppAppProject(mainpkg, "airquality", "airquality")
	airquality.AddDependencies(wifipkg.GetMainLib())
	airquality.AddDependencies(sensorspkg.GetMainLib())
	airquality.AddSourceFiles("common", ".cpp")

	humanpresence := denv.SetupCppAppProject(mainpkg, "humanpresence", "humanpresence")
	humanpresence.AddDependencies(wifipkg.GetMainLib())
	humanpresence.AddDependencies(sensorspkg.GetMainLib())
	humanpresence.AddSourceFiles("common", ".cpp")

	magnet := denv.SetupCppAppProject(mainpkg, "magnet", "magnet")
	magnet.AddDependencies(wifipkg.GetMainLib())
	magnet.AddDependencies(sensorspkg.GetMainLib())
	magnet.AddSourceFiles("common", ".cpp")

	sh1107 := denv.SetupCppAppProjectForArduinoEsp32(mainpkg, "sh1107", "sh1107")
	sh1107.AddDependencies(wifipkg.GetMainLib())
	sh1107.AddDependencies(sensorspkg.GetMainLib())
	sh1107.AddDependencies(u8g2pkg.GetMainLib())
	sh1107.AddSourceFiles("common", ".cpp")

	mg58f18 := denv.SetupCppAppProjectForArduinoEsp32(mainpkg, "mg58f18", "mg58f18")
	mg58f18.AddDependencies(wifipkg.GetMainLib())
	mg58f18.AddDependencies(sensorspkg.GetMainLib())
	mg58f18.AddDependencies(u8g2pkg.GetMainLib())
	mg58f18.AddSourceFiles("common", ".cpp")

	rd03d := denv.SetupCppAppProject(mainpkg, "rd03d", "rd03d")
	rd03d.AddDependencies(wifipkg.GetMainLib())
	rd03d.AddDependencies(sensorspkg.GetMainLib())
	rd03d.AddSourceFiles("common", ".cpp")

	hsp24 := denv.SetupCppAppProject(mainpkg, "hsp24", "hsp24")
	hsp24.AddDependencies(wifipkg.GetMainLib())
	hsp24.AddDependencies(sensorspkg.GetMainLib())
	hsp24.AddSourceFiles("common", ".cpp")

	mainpkg.AddMainApp(airquality)
	mainpkg.AddMainApp(humanpresence)
	mainpkg.AddMainApp(magnet)
	mainpkg.AddMainApp(sh1107)
	mainpkg.AddMainApp(mg58f18)
	mainpkg.AddMainApp(rd03d)
	mainpkg.AddMainApp(hsp24)

	return mainpkg
}
