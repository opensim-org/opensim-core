echo Copying libraries to %RDI_DEV%\LibDev\%RDI_ARCH%[D]

if x%1 == xD GOTO DEBUG
for /r %%i in (Release\*.lib) do copy %%i %RDI_DEV%\LibDev\%RDI_ARCH%
for /r %%i in (Release\*.dll) do copy %%i %RDI_DEV%\LibDev\%RDI_ARCH%
for /r %%i in (Release\*.mll) do copy %%i %RDI_DEV%\LibDev\%RDI_ARCH%
for /r %%i in (Release\*.exe) do copy "%%i" "%RDI_DEV%\LibDev\%RDI_ARCH%"
GOTO END;

:DEBUG
SET RDI_ARCHD=%RDI_ARCH%D
for /r %%i in (Debug\*.lib) do copy %%i %RDI_DEV%\LibDev\%RDI_ARCHD%
for /r %%i in (Debug\*.dll) do copy %%i %RDI_DEV%\LibDev\%RDI_ARCHD%
for /r %%i in (Debug\*.mll) do copy %%i %RDI_DEV%\LibDev\%RDI_ARCHD%
for /r %%i in (Debug\*.exe) do copy "%%i" "%RDI_DEV%\LibDev\%RDI_ARCHD%"

:END
exit(0)

