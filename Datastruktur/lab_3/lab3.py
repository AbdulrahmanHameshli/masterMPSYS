import sys
import os
from collections import OrderedDict
from stopwatch import Stopwatch
import type_checking
from pathlib import Path
from typing import List, Dict, Tuple
from ngram import Ngram, make_ngrams
from hash_table import HashTable

# The main plagiarism detection program.
# You only need to change build_index and find_similarity.
def main():
    if len(sys.argv) <= 1:
        print("Usage: you have to provide a program argument:", file=sys.stderr)
        print("  (1) the name of the directory to scan", file=sys.stderr)
        sys.exit(1)

    # Find all files in the directory and sort the filenames
    paths: List[Path] = list(Path(sys.argv[1]).iterdir())
    paths.sort()

    # Stopwatches time the execution of each phase of the program.
    # You can ignore any code that mentions a stopwatch!
    stopwatch_total = Stopwatch()

    # Read all input files.
    stopwatch = Stopwatch()
    files: Dict[Path, List[Ngram]] = read_paths(paths)
    stopwatch.finished("Reading all input files")
    type_checking.check_files(files)

    # Build index of n-grams (not implemented yet).
    stopwatch = Stopwatch()
    index: Dict[Ngram, List[Path]] = build_index(files)
    stopwatch.finished("Building n-gram index")
    type_checking.check_index(index)

    # Compute similarity of all file pairs.
    stopwatch = Stopwatch()
    similarity: Dict[Tuple[Path, Path], int] = find_similarity(files, index)
    stopwatch.finished("Computing similarity scores")
    type_checking.check_similarity(similarity)

    # Find most similar file pairs, arranged in decreasing order of similarity.
    stopwatch = Stopwatch()
    most_similar: List[Tuple[Path, Path]] = find_most_similar(similarity)
    stopwatch.finished("Finding the most similar files")
    type_checking.check_most_similar(most_similar)
    stopwatch_total.finished("In total the program")
    print()

    # Print out some statistics.
    print("Hash table statistics:")
    print("  files: " + statistics(files))
    print("  index: " + statistics(index))
    print("  similarity: " + statistics(similarity))
    print()

    # Print out the plagiarism report!
    print("Plagiarism report:")
    for doc1, doc2 in most_similar[:50]:
        print("%5d similarity: %s and %s" % (similarity[(doc1, doc2)], doc1.name, doc2.name))

def statistics(x):
    """Get statistics about a hash table."""

    if hasattr(x, "statistics"):
        return x.statistics()
    elif isinstance(x, dict):
        return "dictionary, size %d" % len(x)
    else:
        return "unknown data structure"

def read_paths(paths: List[Path]) -> Dict[Path, List[Ngram]]:
    """Phase 1: Read in each file and chop it into n-grams."""

    files: Dict[Path, List[Ngram]]
    files = HashTable()
    for path in paths:
        contents = open(path,encoding="utf-8").read()
        ngrams = make_ngrams(contents)

        files[path] = ngrams

    return files

def build_index(files: Dict[Path, List[Ngram]]) -> Dict[Ngram, List[Path]]: # O(n)
    """Phase 2: Build index of n-grams (not implemented yet)."""

    index: Dict[Ngram, List[Path]]
    index = HashTable()
    # TODO: build index of n-grams
    
    # for i in files.keys():    """for task 2 """
    for i in files:
        for j in files[i]:
            if j in index:
                index[j].append(i)
            else:
                index[j]=[i]
    return index
   
def find_similarity(files: Dict[Path, List[Ngram]], index: Dict[Ngram, List[Path]]) -> Dict[Tuple[Ngram, Ngram], int]:  # O(n)
    """Phase 3: Count how many n-grams each pair of files has in common."""

    # TODO: Use index to make this loop much more efficient.
    similarity: Dict[Tuple[Ngram, Ngram], int]
    similarity = HashTable()
  
    for n in index:
        for path1 in index[n]:                                                             
            for path2 in index[n]:
                if path1 != path2:
                    if (path1, path2) in similarity:
                        similarity[(path1, path2)] += 1
                    else:
                        similarity[(path1, path2)] = 1

                

            # for ngram1 in files[path1]:
            #     for ngram2 in files[path2]:
            #         if ngram1 == ngram2:
            #             if (path1, path2) in similarity:
            #                 similarity[(path1, path2)] += 1
            #             else:
            #                 similarity[(path1, path2)] = 1

    return similarity

def find_most_similar(similarity: Dict[Tuple[Ngram, Ngram], int]) -> List[Tuple[Ngram, Ngram]]:
    """Phase 4: find all pairs of files with more than 30 n-grams
    in common, sorted in descending order of similarity."""

    # Find all distinct pairs with more than 30 n-grams in common.
    pairs = []
    for file1, file2 in similarity:
        if file1 != file2 and similarity[(file1, file2)] >= 30:
            pair = canonicalise_pair(file1, file2)
            pairs.append(pair)
            # Make sure that similarity[pair] will work
            if pair not in similarity:
                similarity[pair] = similarity[(file1, file2)]
    pairs = remove_duplicates(pairs)

    # Sort to have the most similar pairs first.
    pairs.sort(key = lambda pair: similarity[pair])
    pairs.reverse()
    return pairs

def canonicalise_pair(file1: Path, file2: Path) -> Tuple[Path, Path]:
    """Sort the order of files in a file-pair.
    This is useful when we want (file1, file2) and (file2, file1) to
    be considered equal."""

    if file1 <= file2:
        return file1, file2
    else:
        return file2, file1

def remove_duplicates(x):
    """Remove duplicate items from a list."""

    # Note: you do not need to change this OrderedDict into a HashTable.
    return list(OrderedDict.fromkeys(x))

if __name__ == '__main__':
    main()
