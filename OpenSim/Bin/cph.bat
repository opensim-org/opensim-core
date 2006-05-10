echo Copying include files to %RDI_DEV%\IncludeDev
cd %RDI_DEV%\Source
for /r %%i in (*.h) do copy %%i %RDI_DEV%\IncludeDev
exit(0)





