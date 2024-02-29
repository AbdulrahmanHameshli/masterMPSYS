from term import Term


def first_index_of(array, term, ordering):
    # TODO

    l=0
    h=len(array)-1
    Fisrt_t = -1
    while l<= h:
       
        mid=(h+l)//2
       
        if ordering(array[mid])==ordering(term):
            Fisrt_t=mid 
            h= mid - 1 
        
        elif ordering(array[mid])>ordering(term):
            h= mid - 1 
        
        else:
            l =mid + 1 
    
    return Fisrt_t


def last_index_of(array, term,ordering):
    # TODO
    
    l=0
    h=len(array)-1
    last_t = -1
    while l<= h:
       
        mid=(h+l)//2
       
        if ordering(array[mid])==ordering(term):
            last_t=mid
            l= mid +1 
        
        elif ordering(array[mid])>ordering(term):
            h= mid -1 
        
        else:
            l = mid + 1 
    return last_t
    # raise NotImplementedError()



if __name__ == '__main__':
    # Here you can write some tests if you want.
    t1 = Term("abc", 20)
    t2 = Term("ABCD", 30)
    t3 = Term("bda", 25)
    t0 = Term("ABCD", 30)
    array=[t1,t1,t1,t2,t3,t3]

    print(first_index_of(array,t0,Term.reverse_weight_order))
    print(last_index_of(array,t1,Term.lexicographic_order))

