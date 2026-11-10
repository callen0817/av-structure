echo "Starting Build Process for AV Structure Converter..."
.venv_310\Scripts\pyinstaller --clean converter.spec
echo "Build complete. Check the 'dist/AV_Structure_Converter' directory."
