@echo off

rem -------------------------------------------------
rem jarファイルを起動する
rem -------------------------------------------------

rem ディレクトリの検索
dir/w

:filename
rem 起動ファイル
echo
echo jarファイル名をを入力してください

rem jarファイル名の入力を求める
set /p jarfile=jarファイル：

rem ファイルのありなしで処理を分岐

if exist out.txt (
echo out.txt を削除します。
del out.txt
)

if exist %jarfile% (
rem out.txt の削除
if exist out.txt (
echo out.txt を削除します。
del out.txt
)


@echo on
java -jar %jarfile% 
echo ========計算結果の表示==============
type out.txt
echo =======ここまで===============
@echo off
) else (
@echo off
echo [%jarfile%] がありません。
goto filename
)

@echo off
echo.
echo 終了するには、何かキーを押してください。
pause > nul
