
from term import Term
from range_binary_search import *

class Autocompleter:
    def __init__(self, dictionary):
        """Initializes the dictionary from the given list of terms."""
        self.dictionary = dictionary
        self.sort_dictionary()

    def sort_dictionary(self):
        """Sorts the dictionary in case-insensitive lexicographic order.
        Complexity: O(N log N) where N is the number of dictionary terms."""
     
        self.dictionary.sort(key=Term.lexicographic_order)  
        # raise NotImplementedError()

    def number_of_matches(self, prefix):
        """Returns the number of terms that start with the given prefix.
        Precondition: the internal dictionary is in lexicographic order.
        Complexity: O(log N) where N is the number of dictionary terms."""
        # TODO
        first=first_index_of(self.dictionary, Term(prefix, 0),Term.prefix_order(len(prefix)))
        last=last_index_of(self.dictionary, Term(prefix, 0),Term.prefix_order(len(prefix)))
        if first==-1:
            print("Not in List")
            return 0
        elif last == -1: 
            print("Not in List")
            return 0

        return (last - first +1)
     
    def all_matches(self, prefix):
        """Returns all terms that start with the given prefix, in descending order of weight.
        Precondition: the internal dictionary is in lexicographic order.
        Complexity: O(log N + M log M) where M is the number of matching terms."""
        # TODO

        order = Term.prefix_order(len(prefix))                           #  O(1)                           
        start = first_index_of(self.dictionary, Term(prefix, 0),order)   # O(log n)
        end = last_index_of(self.dictionary, Term(prefix, 0),order)      # O(log n)
        sub_array = self.dictionary[start:end+1]                         #O(1)

        if (start or end) == -1:
            print("Not in lsit")
            return []
        sub_array.sort(key=Term.reverse_weight_order)                    #O(m log m)
   
        return sub_array




# if __name__ == '__main__':
#     # run some tests...
   

#     t2 = Term("ABCD", 30)
#     t3 = Term("bdafsd", 25)

#     dict=[t1,t2,t3]
#     auto = Autocompleter(dict)
    
#     for i in auto.dictionary:
#         print(i)
    
    

    
       
   