
class Term:
    def __init__(self, word, weight):
        self.word = word
        self.weight = weight

    def __str__(self):
        """Returns a string representation of this term in the form WORD:WEIGHT"""
        return f"{self.word}:{self.weight}"

    @staticmethod
    def lexicographic_order(term):
        """Case-insensitive lexicographic order."""
        temp = term.word.casefold()
        return temp   
        # Hint: consider using the casefold method: https://docs.python.org/3/library/stdtypes.html#str.casefold
        # raise NotImplementedError()

    @staticmethod
    def reverse_weight_order(term):
        """Descending order by weight."""
        temp= (-1)*(term.weight)
        return temp

        # raise NotImplementedError()

    @staticmethod
    def prefix_order(k):
        """Case-insensitive lexicographic order, but using only the first k characters of the word."""
        def prefix_k_order(term):
         # TODO
            temp = term.word[:k].casefold()
            return temp
            # raise NotImplementedError()
        return prefix_k_order



######################################################################
# For testing purposes.

# Tests one comparison with the given ordering, and prints the result.
def test_ordering(order, t1, t2, correct):
    result = ("<" if order(t1) < order(t2) else
              ">" if order(t1) > order(t2) else "=")
    print(f"Testing: {t1} {result} {t2}",
          "" if result == correct else f" (ERROR: should be {correct})")

# Runs some simple tests on the orderings.
if __name__ == '__main__':
    t1 = Term("abc", 20)
    t2 = Term("ABCD", 30)
    t3 = Term("bda", 25)
    print("* Testing lexicographic_order:")
    test_ordering(Term.lexicographic_order, t1, t2, "<")
    test_ordering(Term.lexicographic_order, t2, t3, "<")
    test_ordering(Term.lexicographic_order, t3, t1, ">")
    print()
    print("* Testing reverse_weight_order:")

    test_ordering(Term.reverse_weight_order, t1, t2, ">")
    test_ordering(Term.reverse_weight_order, t2, t3, "<")
    test_ordering(Term.reverse_weight_order, t3, t1, "<")
    print()
    print("* Testing prefix_order(3):")
    test_ordering(Term.prefix_order(3), t1, t2, "=")
    test_ordering(Term.prefix_order(3), t2, t3, "<")
    test_ordering(Term.prefix_order(3), t3, t1, ">")
    print()
    print("* Testing prefix_order(2):")
    test_ordering(Term.prefix_order(2), t1, t2, "=")
    test_ordering(Term.prefix_order(2), t2, t3, "=")
    test_ordering(Term.prefix_order(2), t3, t1, "=")
    print() 