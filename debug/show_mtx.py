import scipy.io
import matplotlib.pyplot as plt

matrix_file1 = "debug/debug_fatrop_actual.mtx"
matrix1 = scipy.io.mmread(matrix_file1)
sparse_matrix1 = matrix1.tocsr()

matrix_file2 = "debug/debug_fatrop_expected.mtx"
matrix2 = scipy.io.mmread(matrix_file2)
sparse_matrix2 = matrix2.tocsr()

# overlay the two spy plots
plt.spy(sparse_matrix2, markersize=1, label="Expected")
plt.spy(sparse_matrix1, markersize=1, color='red', label="Actual")
plt.title("Sparsity Pattern of the Matrix")
plt.xlabel("Columns")
plt.ylabel("Rows")
plt.legend()
plt.show()

# Optional: Save the plot
# plt.savefig("sparsity_pattern.png", dpi=300)
