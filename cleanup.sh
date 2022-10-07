echo Delete Build Binaries Saved Intermediate
rm -r Build Binaries Saved Intermediate DerivedDataCache

echo Delete Binaries Intermediate in Plugin
for d in Plugins/* ; do
    rm -r $d/Binaries $d/Intermediate
done

