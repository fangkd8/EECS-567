
// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    heap[heap.length]=new_element;
    var j,temp;
    for(j=heap.length;j>=0;j--){
       if(heap[j]<heap[Math.ceil((j-2)/2)]){
          temp = heap[Math.ceil((j-2)/2)];
          heap[Math.ceil((j-2)/2)] = heap[j];
          heap[j] = temp;
        }
    }
    // console.log("",heap[0]);
    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
    var k,temp,br,kn;
    kn = heap[0];
    temp = heap[heap.length-1];
    heap[heap.length-1] = heap[0];
    heap[0] = temp;
    heap.length -= 1;
    for (k = 0; k < heap.length; k++) {
       if ((2*k + 1<heap.length)&&heap[k]>heap[2*k + 1]) {
          br = heap[k];
          heap[k] = heap[2*k + 1];
          heap[2*k +1] = br;
        }
       if ((2*k + 2<heap.length)&&heap[k]>heap[2*k + 2]) {
          br = heap[k];
          heap[k] = heap[2*k + 2];
          heap[2*k +2] = br;
        }
    }
    return kn;   
    // STENCIL: implement your min binary heap extract operation
}
minheaper.extract = minheap_extract;
// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object






