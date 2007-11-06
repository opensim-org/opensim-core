function a = cellFindAndReplace( a, old, new, eq )

indicesToReplace = find( eq( a, old ) );
for currentIndex = indicesToReplace
    a{ currentIndex } = new;
end
