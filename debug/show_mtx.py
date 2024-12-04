import scipy.io
import matplotlib.pyplot as plt

matrix_file1 = "debug/debug_fatrop_actual.mtx"
# matrix_file1 = "debug/J_sparsity.mtx"
matrix1 = scipy.io.mmread(matrix_file1)
sparse_matrix1 = matrix1.tocsr()

matrix_file2 = "debug/debug_fatrop_expected.mtx"
# matrix_file2 = "debug/J_sparsity.mtx"
matrix2 = scipy.io.mmread(matrix_file2)
sparse_matrix2 = matrix2.tocsr()

# overlay the two spy plots
plt.spy(sparse_matrix2, markersize=3, label="Expected")
plt.spy(sparse_matrix1, markersize=1, color='red', label="Actual")
plt.title("Sparsity Pattern of constraint Jacobian")
plt.xlabel("Columns")
plt.ylabel("Rows")

# put a legend to the right of the plot
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.show()

# Optional: Save the plot
# plt.savefig("sparsity_pattern.png", dpi=300)
