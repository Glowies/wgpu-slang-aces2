for file in *.tiff; do
    oiiotool "$file" -o "${file%.tiff}.exr"
done
