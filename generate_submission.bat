
set TEMPDIR="%TEMP%.\cs_c3100_submit"
del /f /s /q %TEMPDIR% 1>nul
rmdir /s /q %TEMPDIR%
mkdir %TEMPDIR%

xcopy /q /s src "%TEMP%.\cs_c3100_submit\src\"
xcopy /q /s assets "%TEMP%.\cs_c3100_submit\assets\"
xcopy /q assignment.sln %TEMPDIR%
xcopy /q assignment.vcxproj %TEMPDIR%
xcopy /q assignment.vcxproj.filters %TEMPDIR%
xcopy /q framework.vcxproj %TEMPDIR%
xcopy /q framework.vcxproj.filters %TEMPDIR%
xcopy /q README.txt %TEMPDIR%

set SCRIPTFILE="%TEMP%.\cs_c3100_generate_submission.vbs"

@echo off
echo Set objArgs = WScript.Arguments > %SCRIPTFILE%
echo Set FS = CreateObject("Scripting.FileSystemObject") >> %SCRIPTFILE%
echo InputFolder = FS.GetAbsolutePathName(objArgs(0)) >> %SCRIPTFILE%
echo ZipFile = FS.GetAbsolutePathName(objArgs(1)) >> %SCRIPTFILE%
echo FS.CreateTextFile(ZipFile, True).Write "PK" ^& Chr(5) ^& Chr(6) ^& String(18, vbNullChar) >> %SCRIPTFILE%
echo Set objShell = CreateObject("Shell.Application") >> %SCRIPTFILE%
echo Set source = objShell.NameSpace(InputFolder).Items >> %SCRIPTFILE%
echo objShell.NameSpace(ZipFile).CopyHere(source) >> %SCRIPTFILE%
echo Set scriptShell = CreateObject("Wscript.Shell") >> %SCRIPTFILE%
echo Do While scriptShell.AppActivate("Compressing...") = FALSE >> %SCRIPTFILE%
echo    WScript.Sleep 50 >> %SCRIPTFILE%
echo Loop >> %SCRIPTFILE%
echo Do While scriptShell.AppActivate("Compressing...") = TRUE >> %SCRIPTFILE%
echo    WScript.Sleep 50 >> %SCRIPTFILE%
echo Loop >> %SCRIPTFILE%
@echo on

CScript %SCRIPTFILE% %TEMPDIR% assignment3.zip

del %SCRIPTFILE%
del /f /s /q %TEMPDIR% 1>nul
rmdir /s /q %TEMPDIR%
